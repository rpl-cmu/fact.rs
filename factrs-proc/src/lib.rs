use syn::{parse_macro_input, ItemImpl};

mod fac;
mod noise;
mod residual;
mod robust;
mod variable;

enum BoxedTypes {
    Residual,
    Variable,
    Noise,
    Robust,
}

fn check_type(input: &ItemImpl) -> syn::Result<BoxedTypes> {
    let err = syn::Error::new_spanned(input, "Missing trait");
    let result = &input
        .trait_
        .as_ref()
        .ok_or(err.clone())?
        .1
        .segments
        .last()
        .ok_or(err)?
        .ident
        .to_string();

    if result.contains("Residual") {
        Ok(BoxedTypes::Residual)
    } else if result.contains("Variable") {
        Ok(BoxedTypes::Variable)
    } else if result.contains("Noise") {
        Ok(BoxedTypes::Noise)
    } else if result.contains("Robust") {
        Ok(BoxedTypes::Robust)
    } else {
        Err(syn::Error::new_spanned(input, "Not a valid trait"))
    }
}

/// Mark an implementation of [Variable](factrs::traits::Variable),
/// [Residual](factrs::traits::Residual), [Noise](factrs::traits::Noise), or
/// [Robust](factrs::traits::Robust).
///
/// This is mostly to aid in serialization when the `serde` feature is enabled.
/// Since these items are boxed inside [Factor](factrs::core::Factor), we use
/// `typetag` for serialization which requires a little bit of manual work.
///
/// For examples of usage, check out the [tests](https://github.com/rpl-cmu/factrs/tree/dev/tests) folder.
///
/// Specifically, it does the following for each trait:
///
/// ### [Variable](factrs::traits::Variable)
/// If serde is disabled, does nothing. Otherwise, it does the following:
/// - If the only generic is the datatype (and potentially a const usize
///   generic), add tag for serialization
/// - Add tag for serializing [PriorResidual<Type>](factrs::core::PriorResidual)
///   and [BetweenResidual<Type>](factrs::core::BetweenResidual) as well.
///
/// ### [Residual](factrs::traits::Residual)
/// This should be applied on a numbered residual such as
/// [Residual2](factrs::core::Residual2). It will automatically derive
/// [Residual](factrs::traits::Residual) in this case. Additionally, if serde is
/// enabled, it will add a tag for serialization.
///
/// ### [Noise](factrs::traits::Noise)
/// If serde is disabled, does nothing. Otherwise, it will tag the noise model
/// for serialization, up to size 20.
///
/// ### [Robust](factrs::traits::Robust)
/// If serde is disabled, does nothing. Otherwise, it will tag the robust
/// kernel.
#[proc_macro_attribute]
pub fn mark(
    _args: proc_macro::TokenStream,
    input: proc_macro::TokenStream,
) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as ItemImpl);

    let trait_type = match check_type(&input) {
        Ok(syntax_tree) => syntax_tree,
        Err(err) => return err.to_compile_error().into(),
    };

    match trait_type {
        BoxedTypes::Residual => residual::mark(input),
        BoxedTypes::Variable => variable::mark(input),
        BoxedTypes::Noise => noise::mark(input),
        BoxedTypes::Robust => robust::mark(input),
    }
    .into()
}

/// Easiest way to create a factor
///
/// Similar to `vec!` in the std library, this is a macro to create a factor
/// from a residual, keys, and alternatively noise and robust kernel. A simple
/// usage would be
/// ```
/// # use factrs::{assign_symbols, fac, core::{SO2, PriorResidual, BetweenResidual}, traits::*};
/// # let prior = BetweenResidual::new(SO2::identity());
/// # let between = PriorResidual::new(SO2::identity());
/// # assign_symbols!(X: SO2);
/// let prior_factor = fac![prior, X(0)];
/// let between_factor = fac![between, (X(0), X(1))];
/// ```
/// Additionally, there is a number of helper options for specifying a noise
/// model,
/// ```
/// # use factrs::{assign_symbols, fac, core::{SO2, PriorResidual, GaussianNoise}, traits::*};
/// # let prior = PriorResidual::new(SO2::identity());
/// # assign_symbols!(X: SO2);
/// let noise = GaussianNoise::from_scalar_sigma(0.1);
/// let f1a = fac![prior, X(0), noise];
/// let f1b = fac![prior, X(0), 0.1 as std];
/// let f2 = fac![prior, X(0), 0.1 as cov];
/// let f3 = fac![prior, X(0), (0.1, 0.3) as std];
/// ```
/// where `f1a` and `f1b` are identical, and where `f3` uses
/// [GaussianNoise::from_split_sigma](factrs::noise::GaussianNoise::from_split_sigma)
/// to specify the rotation and translation noise separately. (where rotation is
/// ALWAYS first in factrs)
///
/// Finally, a robust kernel can be specified as well,
/// ```
/// # use factrs::{assign_symbols, fac, core::{SO2, PriorResidual, Huber}, traits::*};
/// # let prior = PriorResidual::new(SO2::identity());
/// # assign_symbols!(X: SO2);
/// let f1 = fac![prior, X(0), 0.1 as std, Huber::default()];
/// let f2 = fac![prior, X(0), _, Huber::default()];
/// ```
/// where `f2` uses [UnitNoise](factrs::noise::UnitNoise) as the noise model.
#[proc_macro]
pub fn fac(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let factor = parse_macro_input!(input as fac::Factor);

    fac::fac(factor).into()
}
